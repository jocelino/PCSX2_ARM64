function(detect_operating_system)
	message(STATUS "CMake Version: ${CMAKE_VERSION}")
	message(STATUS "CMake System Name: ${CMAKE_SYSTEM_NAME}")

	# LINUX wasn't added until CMake 3.25.
	if (CMAKE_VERSION VERSION_LESS 3.25.0 AND CMAKE_SYSTEM_NAME MATCHES "Linux")
		# Have to make it visible in this scope as well for below.
		set(LINUX TRUE PARENT_SCOPE)
		set(LINUX TRUE)
	endif()

	if(WIN32)
		message(STATUS "Building for Windows.")
	elseif(APPLE AND NOT IOS)
		message(STATUS "Building for MacOS.")
	elseif(LINUX)
		message(STATUS "Building for Linux.")
	elseif(ANDROID)
		message(STATUS "Building for Android.")
	elseif(BSD)
		message(STATUS "Building for *BSD.")
	else()
		message(FATAL_ERROR "Unsupported platform.")
	endif()
endfunction()

function(detect_compiler)
	if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
		set(USE_CLANG_CL TRUE PARENT_SCOPE)
		set(IS_SUPPORTED_COMPILER TRUE PARENT_SCOPE)
		message(STATUS "Building with Clang-CL.")
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
		set(USE_CLANG TRUE PARENT_SCOPE)
		set(IS_SUPPORTED_COMPILER TRUE PARENT_SCOPE)
		message(STATUS "Building with Clang/LLVM.")
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(USE_GCC TRUE PARENT_SCOPE)
		set(IS_SUPPORTED_COMPILER FALSE PARENT_SCOPE)
		message(STATUS "Building with GNU GCC.")
	elseif(MSVC)
		set(IS_SUPPORTED_COMPILER TRUE PARENT_SCOPE)
		message(STATUS "Building with MSVC.")
	else()
		message(FATAL_ERROR "Unknown compiler: ${CMAKE_CXX_COMPILER_ID}")
	endif()
endfunction()

function(get_git_version_info)
	set(PCSX2_GIT_REV "")
	set(PCSX2_GIT_TAG "")
	set(PCSX2_GIT_HASH "")
	if (GIT_FOUND AND EXISTS ${PROJECT_SOURCE_DIR}/.git)
		EXECUTE_PROCESS(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} describe --tags
			OUTPUT_VARIABLE PCSX2_GIT_REV
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET)

		EXECUTE_PROCESS(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} tag --points-at HEAD --sort=version:refname
			OUTPUT_VARIABLE PCSX2_GIT_TAG_LIST
			RESULT_VARIABLE TAG_RESULT
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET)

		# CAUTION: There is a race here, this solves the problem of a commit being tagged multiple times (take the last tag)
		# however, if simultaneous builds are pushing tags to the same commit you might get inconsistent results (it's a race)
		#
		# The easy solution is, don't do that, but just something to be aware of.
		if(PCSX2_GIT_TAG_LIST AND TAG_RESULT EQUAL 0)
			string(REPLACE "\n" ";" PCSX2_GIT_TAG_LIST "${PCSX2_GIT_TAG_LIST}")
			if (PCSX2_GIT_TAG_LIST)
				list(GET PCSX2_GIT_TAG_LIST -1 PCSX2_GIT_TAG)
				message("Using tag: ${PCSX2_GIT_TAG}")
			endif()
		endif()

		EXECUTE_PROCESS(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
			OUTPUT_VARIABLE PCSX2_GIT_HASH
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET)

		EXECUTE_PROCESS(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=local
			OUTPUT_VARIABLE PCSX2_GIT_DATE
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET)
	endif()
	if (NOT PCSX2_GIT_REV)
		EXECUTE_PROCESS(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
			OUTPUT_VARIABLE PCSX2_GIT_REV
			OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET)
		if (NOT PCSX2_GIT_REV)
			set(PCSX2_GIT_REV "Unknown")
		endif()
	endif()

	set(PCSX2_GIT_REV "${PCSX2_GIT_REV}" PARENT_SCOPE)
	set(PCSX2_GIT_TAG "${PCSX2_GIT_TAG}" PARENT_SCOPE)
	set(PCSX2_GIT_HASH "${PCSX2_GIT_HASH}" PARENT_SCOPE)
	set(PCSX2_GIT_DATE "${PCSX2_GIT_DATE}" PARENT_SCOPE)
endfunction()

function(write_svnrev_h)
	if ("${PCSX2_GIT_TAG}" MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
		file(WRITE ${CMAKE_BINARY_DIR}/common/include/svnrev.h
			"#define GIT_TAG \"${PCSX2_GIT_TAG}\"\n"
			"#define GIT_TAGGED_COMMIT 1\n"
			"#define GIT_TAG_HI  ${CMAKE_MATCH_1}\n"
			"#define GIT_TAG_MID ${CMAKE_MATCH_2}\n"
			"#define GIT_TAG_LO  ${CMAKE_MATCH_3}\n"
			"#define GIT_REV \"${PCSX2_GIT_TAG}\"\n"
			"#define GIT_HASH \"${PCSX2_GIT_HASH}\"\n"
			"#define GIT_DATE \"${PCSX2_GIT_DATE}\"\n"
		)
	elseif ("${PCSX2_GIT_REV}" MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)")
		file(WRITE ${CMAKE_BINARY_DIR}/common/include/svnrev.h
			"#define GIT_TAG \"${PCSX2_GIT_TAG}\"\n"
			"#define GIT_TAGGED_COMMIT 0\n"
			"#define GIT_TAG_HI  ${CMAKE_MATCH_1}\n"
			"#define GIT_TAG_MID ${CMAKE_MATCH_2}\n"
			"#define GIT_TAG_LO  ${CMAKE_MATCH_3}\n"
			"#define GIT_REV \"${PCSX2_GIT_REV}\"\n"
			"#define GIT_HASH \"${PCSX2_GIT_HASH}\"\n"
			"#define GIT_DATE \"${PCSX2_GIT_DATE}\"\n"
		)
	else()
		file(WRITE ${CMAKE_BINARY_DIR}/common/include/svnrev.h
			"#define GIT_TAG \"${PCSX2_GIT_TAG}\"\n"
			"#define GIT_TAGGED_COMMIT 0\n"
			"#define GIT_TAG_HI 0\n"
			"#define GIT_TAG_MID 0\n"
			"#define GIT_TAG_LO 0\n"
			"#define GIT_REV \"${PCSX2_GIT_REV}\"\n"
			"#define GIT_HASH \"${PCSX2_GIT_HASH}\"\n"
			"#define GIT_DATE \"${PCSX2_GIT_DATE}\"\n"
		)
	endif()
endfunction()

function(check_no_parenthesis_in_path)
	if ("${CMAKE_BINARY_DIR}" MATCHES "[()]" OR "${CMAKE_SOURCE_DIR}" MATCHES "[()]")
		message(FATAL_ERROR "Your path contains some parenthesis. Unfortunately Cmake doesn't support them correctly.\nPlease rename your directory to avoid '(' and ')' characters\n")
	endif()
endfunction()

# like add_library(new ALIAS old) but avoids add_library cannot create ALIAS target "new" because target "old" is imported but not globally visible. on older cmake
function(alias_library new old)
	string(REPLACE "::" "" library_no_namespace ${old})
	if (NOT TARGET _alias_${library_no_namespace})
		add_library(_alias_${library_no_namespace} INTERFACE)
		target_link_libraries(_alias_${library_no_namespace} INTERFACE ${old})
	endif()
	add_library(${new} ALIAS _alias_${library_no_namespace})
endfunction()

function(source_groups_from_vcxproj_filters file)
	file(READ "${file}" filecontent)
	get_filename_component(parent "${file}" DIRECTORY)
	if (parent STREQUAL "")
		set(parent ".")
	endif()
	set(regex "<[^ ]+ Include=\"([^\"]+)\">[ \t\r\n]+<Filter>([^<]+)<\\/Filter>[ \t\r\n]+<\\/[^ >]+>")
	string(REGEX MATCHALL "${regex}" filterstrings "${filecontent}")
	foreach(filterstring IN LISTS filterstrings)
		string(REGEX REPLACE "${regex}" "\\1" path "${filterstring}")
		string(REGEX REPLACE "${regex}" "\\2" group "${filterstring}")
		source_group("${group}" FILES "${parent}/${path}")
	endforeach()
endfunction()

# Extracts a translation with the given type ("source" or "translation") from the given category of the given ts file
# (If there's multiple strings under the same category, which one it extracts is implementation defined.  Just don't do it.)
function(extract_translation_from_ts file type category output)
	file(READ "${file}" filecontent)
	# Don't you love it when the best parser your language has to offer is regex?
	set(regex_search "(<[^\\/>]+>[^<>]+<\\/[^>\\/]+>|<\\/?context>)")
	set(regex_extract "<[^\\/>]+>([^<>]+)<\\/([^>\\/]+)>")
	string(REGEX MATCHALL "${regex_search}" pieces "${filecontent}")
	foreach(piece IN LISTS pieces)
		if (piece STREQUAL "<context>")
			set(found "")
			set(name_match FALSE)
		elseif (piece STREQUAL "</context>")
			if (name_match)
				set(${output} "${found}" PARENT_SCOPE)
				break()
			endif()
		else()
			string(REGEX REPLACE "${regex_extract}" "\\1" content "${piece}")
			string(REGEX REPLACE "${regex_extract}" "\\2" tag "${piece}")
			if (tag STREQUAL "name" AND content STREQUAL "${category}")
				set(name_match TRUE)
			endif()
			if (tag STREQUAL "${type}")
				set(found "${content}")
			endif()
		endif()
	endforeach()
endfunction()

function(fixup_file_properties target)
	get_target_property(SOURCES ${target} SOURCES)
	if(APPLE)
		foreach(source IN LISTS SOURCES)
			# Set the right file types for .inl files in Xcode
			if("${source}" MATCHES "\\.(inl|h)$")
				set_source_files_properties("${source}" PROPERTIES XCODE_EXPLICIT_FILE_TYPE sourcecode.cpp.h)
			endif()
			if("${source}" MATCHES "\\.(qm)$")
				set_source_files_properties("${source}" PROPERTIES XCODE_EXPLICIT_FILE_TYPE compiled)
			endif()
			# CMake makefile and ninja generators will attempt to share one PCH for both cpp and mm files
			# That's not actually OK
			if("${source}" MATCHES "\\.mm$")
				set_source_files_properties("${source}" PROPERTIES SKIP_PRECOMPILE_HEADERS ON)
			endif()
		endforeach()
	endif()
endfunction()

function(disable_compiler_warnings_for_target target)
	if(MSVC)
		target_compile_options(${target} PRIVATE "/W0")
	else()
		target_compile_options(${target} PRIVATE "-w")
	endif()
endfunction()

function(detect_page_size)
	message(STATUS "Determining host page size")
	set(detect_page_size_file ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c)
	file(WRITE ${detect_page_size_file} "
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
int main() {
	int res = sysconf(_SC_PAGESIZE);
	printf(\"%d\", res);
	return (res > 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}")
	try_run(
		detect_page_size_run_result
		detect_page_size_compile_result
		${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}
		${detect_page_size_file}
		RUN_OUTPUT_VARIABLE detect_page_size_output)
	if(NOT detect_page_size_compile_result OR NOT detect_page_size_run_result EQUAL 0 OR CMAKE_CROSSCOMPILING)
		message(FATAL_ERROR "Could not determine host page size.")
	else()
		message(STATUS "Host page size: ${detect_page_size_output}")
		set(HOST_PAGE_SIZE ${detect_page_size_output} CACHE STRING "Reported host page size")
	endif()
endfunction()

function(detect_cache_line_size)
	message(STATUS "Determining host cache line size")
	set(detect_cache_line_size_file ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeTmp/src.c)
	file(WRITE ${detect_cache_line_size_file} "
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
int main() {
	int l1i = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
	int l1d = sysconf(_SC_LEVEL1_ICACHE_LINESIZE);
	int res = (l1i > l1d) ? l1i : l1d;
	for (int index = 0; index < 16; index++) {
		char buf[128];
		snprintf(buf, sizeof(buf), \"/sys/devices/system/cpu/cpu0/cache/index%d/coherency_line_size\", index);
		FILE* fp = fopen(buf, \"rb\");
		if (!fp)
			break;
		fread(buf, sizeof(buf), 1, fp);
		fclose(fp);
		int val = atoi(buf);
		res = (val > res) ? val : res;
	}
	printf(\"%d\", res);
	return (res > 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}")
	try_run(
		detect_cache_line_size_run_result
		detect_cache_line_size_compile_result
		${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}
		${detect_cache_line_size_file}
		RUN_OUTPUT_VARIABLE detect_cache_line_size_output)
	if(NOT detect_cache_line_size_compile_result OR NOT detect_cache_line_size_run_result EQUAL 0 OR CMAKE_CROSSCOMPILING)
		message(FATAL_ERROR "Could not determine host cache line size.")
	else()
		message(STATUS "Host cache line size: ${detect_cache_line_size_output}")
		set(HOST_CACHE_LINE_SIZE ${detect_cache_line_size_output} CACHE STRING "Reported host cache line size")
	endif()
endfunction()

function(get_recursive_include_directories output target inc_prop link_prop)
	get_target_property(dirs ${target} ${inc_prop})
	if(NOT dirs)
		set(dirs)
	endif()
	get_target_property(deps ${target} ${link_prop})
	if(deps)
		foreach(dep IN LISTS deps)
			if(TARGET ${dep})
				get_recursive_include_directories(depdirs ${dep} INTERFACE_INCLUDE_DIRECTORIES INTERFACE_LINK_LIBRARIES)
				foreach(depdir IN LISTS depdirs)
					# Only match absolute paths
					# We'll hope any non-absolute paths will not get set as system directories
					if(depdir MATCHES "^/")
						list(APPEND dirs ${depdir})
					endif()
				endforeach()
			endif()
		endforeach()
		list(REMOVE_DUPLICATES dirs)
	endif()
	set(${output} "${dirs}" PARENT_SCOPE)
endfunction()

function(force_include_last_impl target include inc_prop link_prop)
	get_recursive_include_directories(dirs ${target} ${inc_prop} ${link_prop})
	set(remove)
	foreach(dir IN LISTS dirs)
		if("${dir}" MATCHES "${include}")
			list(APPEND remove ${dir})
		endif()
	endforeach()
	if(NOT "${remove}" STREQUAL "")
		get_target_property(sysdirs ${target} INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
		if(NOT sysdirs)
			set(sysdirs)
		endif()
		# Move matching items to the end
		list(REMOVE_ITEM dirs ${remove})
		list(APPEND dirs ${remove})
		# Set them as system include directories
		list(APPEND sysdirs ${remove})
		list(REMOVE_DUPLICATES sysdirs)
		set_target_properties(${target} PROPERTIES
			${inc_prop} "${dirs}"
			INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${sysdirs}"
		)
	endif()
endfunction()

function(force_include_last target include)
	force_include_last_impl(${target} "${include}" INTERFACE_INCLUDE_DIRECTORIES INTERFACE_LINK_LIBRARIES)
	force_include_last_impl(${target} "${include}" INCLUDE_DIRECTORIES LINK_LIBRARIES)
endfunction()
