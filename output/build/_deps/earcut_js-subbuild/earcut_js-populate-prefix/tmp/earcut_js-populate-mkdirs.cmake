# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-src"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-build"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/tmp"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/src/earcut_js-populate-stamp"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/src"
  "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/src/earcut_js-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/src/earcut_js-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ys/feedback/uidreczz28Z7ClbdazQ/output/build/_deps/earcut_js-subbuild/earcut_js-populate-prefix/src/earcut_js-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
