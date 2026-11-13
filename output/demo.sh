#!/usr/bin/env bash
# Segmented verification demo for the earcut input-validation work.
# Each segment prints a title banner and pauses afterwards (for the recording);
# the script never waits for keyboard input. Artifacts land in output/.
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD=output/build

banner() {
    echo
    echo "================================================================"
    echo "  $1"
    echo "================================================================"
    echo
}

pause() { sleep 2; }

banner "段 1/4：构建（cmake + ninja，-Werror 零警告）"
cmake -B "$BUILD" -GNinja -DEARCUT_BUILD_BENCH=OFF -DEARCUT_BUILD_VIZ=OFF \
      -DEARCUT_WARNING_IS_ERROR=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD"
pause

banner "段 2/4：脏数据诊断演示（自交/重复点/极短边/坐标溢出）"
"$BUILD/fuzz" --demo | tee output/dirty_batch_report.txt
pause

banner "段 3/4：fuzz 驱动 10 万次无崩溃"
"$BUILD/fuzz" 100000 | tee output/fuzz_summary.txt
pause

banner "段 4/4：回归测试（合法输入逐顶点一致 + 全量 ctest）"
"$BUILD/tests" --gtest_filter='EarcutValidate*'
(cd "$BUILD" && ctest --output-on-failure) | tee output/ctest.log
pause

banner "完成：产物见 output/（recording.cast / *.txt / ctest.log）"
