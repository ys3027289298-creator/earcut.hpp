# CMake generated Testfile for 
# Source directory: /home/ys/feedback/uidreczz28Z7ClbdazQ
# Build directory: /home/ys/feedback/uidreczz28Z7ClbdazQ/output/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(earcut_tests "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/tests")
set_tests_properties(earcut_tests PROPERTIES  _BACKTRACE_TRIPLES "/home/ys/feedback/uidreczz28Z7ClbdazQ/CMakeLists.txt;124;add_test;/home/ys/feedback/uidreczz28Z7ClbdazQ/CMakeLists.txt;0;")
add_test(earcut_fuzz "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/fuzz" "100000")
set_tests_properties(earcut_fuzz PROPERTIES  _BACKTRACE_TRIPLES "/home/ys/feedback/uidreczz28Z7ClbdazQ/CMakeLists.txt;131;add_test;/home/ys/feedback/uidreczz28Z7ClbdazQ/CMakeLists.txt;0;")
subdirs("_deps/googletest-build")
