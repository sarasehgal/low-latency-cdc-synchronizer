#!/usr/bin/env bash
# Run the 2x2 NoC top-level traffic testbench (noc_top_tb.sv) in ModelSim.
# Usage: ./run_noc_top.sh [all|single|multi]     (default: all)
#   single : all four routers on one clock
#   multi  : clk, clk*0.99, clk*0.97, clk*0.7
# Optional environment:
#   NUM_FLITS=200  FIFO_DEPTH=16  DATA_W=3  BASE_PERIOD=10.0  SEED=1  SIM_TIMEOUT=600
set -euo pipefail

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
project_dir="$(dirname -- "$script_path")"

if [[ "${1:-}" != "--modelsim-env" ]]; then
    case "${1:-all}" in
        all|single|multi) export NOC_TOP_MODE="${1:-all}" ;;
        -h|--help) sed -n '2,8p' "$script_path"; exit 0 ;;
        *) printf 'Usage: %s [all|single|multi]\n' "$0" >&2; exit 2 ;;
    esac
    export NOC_TOP_RUNNER="$script_path"
    exec /usr/bin/tcsh -f -c \
        'source /CMC/tools/CSHRCs/Mentor.Modelsim10.7c; if ($status != 0) exit 1; exec /bin/bash "$NOC_TOP_RUNNER" --modelsim-env'
fi

case "${NOC_TOP_MODE:-all}" in
    all)    modes=(single multi) ;;
    single) modes=(single) ;;
    multi)  modes=(multi) ;;
esac

sources=(mpam_sync.sv local_fifo.sv router_logic.v router.sv noc_top.sv noc_top_tb.sv)
for f in "${sources[@]}"; do
    [[ -s "$project_dir/$f" ]] || { printf 'ERROR: missing %s/%s\n' "$project_dir" "$f" >&2; exit 1; }
done

build_dir="$(mktemp -d "$project_dir/.modelsim_top_XXXXXX")"
printf 'Build directory: %s\n' "$build_dir"
trap 'printf "\nLogs and waveforms: %s\n" "$build_dir"' EXIT
cd "$build_dir"

vlib work >/dev/null
vmap -c >/dev/null
vlog -sv -work work "${sources[@]/#/$project_dir/}" 2>&1 | tee compile.log
if grep -Eq '\*\* Error' compile.log; then
    printf 'FAIL: compile errors\n' >&2
    exit 1
fi

params=(-gNUM_FLITS="${NUM_FLITS:-200}" -gFIFO_DEPTH="${FIFO_DEPTH:-16}"
        -gDATA_W="${DATA_W:-3}" -gBASE_PERIOD="${BASE_PERIOD:-10.0}")

failed=0
for mode in "${modes[@]}"; do
    multi=0; [[ "$mode" == multi ]] && multi=1
    printf '\n========== %s clock domain(s) ==========\n' "$mode"
    sim_status=0
    timeout "${SIM_TIMEOUT:-600}" \
        vsim -c -onfinish stop -voptargs=+acc -sv_seed "${SEED:-1}" \
            -gMULTI_CLK="$multi" "${params[@]}" \
            -l "sim_${mode}.log" -wlf "noc_top_${mode}.wlf" work.tb_noc_top \
            -do 'onerror {quit -code 1}; log -r /*; run -all; quit -code 0' \
            > "console_${mode}.log" 2>&1 || sim_status=$?
    sed -n '/tb_noc_top:/,/RESULT/p; /\[FAIL\]/p' "sim_${mode}.log" | sed 's/^# //' | head -60
    if (( sim_status != 0 )) \
        || grep -Eq '\[FAIL\]|RESULT: FAIL|\*\* (Error|Fatal)' "sim_${mode}.log" \
        || ! grep -Fq 'RESULT: PASS' "sim_${mode}.log"; then
        printf 'FAIL: %s (simulator exit status %s)\n' "$mode" "$sim_status" >&2
        failed=1
    else
        printf 'PASS: %s; waveform: %s/noc_top_%s.wlf\n' "$mode" "$build_dir" "$mode"
    fi
done
exit "$failed"
