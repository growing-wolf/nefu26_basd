# FetchContent helper: fetch dependencies with automatic local caching
#
# Usage:
#   # By version tag (GitHub)
#   fetch_dependency(NAME fmt REPO fmtlib/fmt VERSION 12.1.0)
#
#   # By version tag (GitLab)
#   fetch_dependency(NAME eigen3 REPO gitlab.com/libeigen/eigen VERSION 5.0.1)
#
#   # By commit hash
#   fetch_dependency(NAME fmt REPO fmtlib/fmt COMMIT abc123)
#
#   # By git (supports local repo)
#   fetch_dependency(NAME fmt GIT_REPOSITORY https://github.com/fmtlib/fmt.git GIT_TAG 12.1.0)
#
#   # Custom ZIP URL
#   fetch_dependency(NAME foo ZIP_URL https://... ZIP_NAME foo.zip)
#
# All downloaded files are cached to 3dparty/ for offline builds.

include(FetchContent)

# =============================================================================
# Private helpers
# =============================================================================

# CMake < 3.25 has no SYSTEM keyword on FetchContent_Declare. Emulate it:
# append include dirs exported by fetched content (build/_deps/...) to the
# SYSTEM include dirs so third-party headers don't trip this project's
# -Werror flags.
#
# Notes:
# - BUILDSYSTEM_TARGETS does not include targets from subdirectories, and
#   FetchContent adds the fetched content via add_subdirectory, so we collect
#   targets recursively via the SUBDIRECTORIES property.
# - The dir must stay in INTERFACE_INCLUDE_DIRECTORIES too (that's what a
#   native PUBLIC SYSTEM does); removing it breaks -isystem propagation
#   through INTERFACE targets on CMake 3.22.
function(_collect_dir_targets_recursive dir out_var)
    set(_targets "")
    get_property(_subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
    foreach(_sd IN LISTS _subdirs)
        get_property(_t DIRECTORY "${_sd}" PROPERTY BUILDSYSTEM_TARGETS)
        if(_t)
            list(APPEND _targets ${_t})
        endif()
        _collect_dir_targets_recursive("${_sd}" _sub_targets)
        if(_sub_targets)
            list(APPEND _targets ${_sub_targets})
        endif()
    endforeach()
    set(${out_var} ${_targets} PARENT_SCOPE)
endfunction()

function(_mark_fetched_content_system)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.25")
        return()
    endif()

    set(_dir_targets "")
    get_property(_own DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
    if(_own)
        list(APPEND _dir_targets ${_own})
    endif()
    _collect_dir_targets_recursive("${CMAKE_CURRENT_SOURCE_DIR}" _sub_targets)
    if(_sub_targets)
        list(APPEND _dir_targets ${_sub_targets})
    endif()
    list(REMOVE_DUPLICATES _dir_targets)

    foreach(_t IN LISTS _dir_targets)
        if(NOT TARGET "${_t}")
            continue()
        endif()
        get_target_property(_is_alias "${_t}" ALIASED_TARGET)
        if(_is_alias)
            continue()
        endif()

        set(_system "")
        get_target_property(_incs "${_t}" INTERFACE_INCLUDE_DIRECTORIES)
        foreach(_inc IN LISTS _incs)
            if(_inc MATCHES "${CMAKE_BINARY_DIR}/_deps")
                list(APPEND _system "${_inc}")
            endif()
        endforeach()
        if(_system)
            get_target_property(_prev_system "${_t}" INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
            if(_prev_system)
                list(APPEND _system ${_prev_system})
            endif()
            list(REMOVE_DUPLICATES _system)
            set_property(TARGET "${_t}" PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_system}")
        endif()
    endforeach()
endfunction()

function(_download_and_cache zip_url zip_name)
    set(local_zip "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/${zip_name}")

    if(EXISTS "${local_zip}")
        # Check for corrupted zero-byte file
        file(SIZE "${local_zip}" file_size)
        if(file_size EQUAL 0)
            message(WARNING "Cached file ${local_zip} is corrupted (0 bytes), re-downloading...")
            file(REMOVE "${local_zip}")
        else()
            # Validate archive integrity so an interrupted download (Ctrl+C)
            # doesn't poison the cache with a truncated archive.
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar tf "${local_zip}"
                OUTPUT_QUIET ERROR_QUIET
                RESULT_VARIABLE tar_result
            )
            if(tar_result EQUAL 0)
                return()
            endif()
            message(WARNING "Cached file ${local_zip} is corrupted (truncated archive), re-downloading...")
            file(REMOVE "${local_zip}")
        endif()
    endif()

    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3dparty")

    message(STATUS "Downloading ${zip_name} to 3dparty/")
    file(DOWNLOAD "${zip_url}" "${local_zip}" STATUS status LOG log)

    list(GET status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download ${zip_url}:\n${log}")
    endif()

    message(STATUS "Cached to: ${local_zip}")
endfunction()

function(_fetch_dependency_from_zip name zip_url zip_name)
    _download_and_cache("${zip_url}" "${zip_name}")
    set(local_zip "${CMAKE_CURRENT_SOURCE_DIR}/3dparty/${zip_name}")
    message(STATUS "Using cached ${name}: ${local_zip}")
    # CMake < 3.24 rejects bare paths in URL; use an explicit file:// scheme.
    # SYSTEM is only understood by FetchContent_Declare since CMake 3.25; on
    # older CMake it leaks into ExternalProject_Add and gets appended to the
    # URL value ("<path>;SYSTEM"), failing URL validation.
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.25")
        FetchContent_Declare(${name} URL "file://${local_zip}" SYSTEM)
    else()
        FetchContent_Declare(${name} URL "file://${local_zip}")
    endif()
    FetchContent_MakeAvailable(${name})
    _mark_fetched_content_system()
endfunction()

function(_fetch_dependency_from_git name repo tag)
    if(EXISTS "${repo}")
        message(STATUS "Using local git repo for ${name}: ${repo}")
    else()
        message(STATUS "Cloning ${name} from ${repo} (tag: ${tag})")
    endif()

    # Same SYSTEM version guard as the zip path above
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.25")
        FetchContent_Declare(${name}
            GIT_REPOSITORY ${repo}
            GIT_TAG ${tag}
            GIT_SHALLOW TRUE
            SYSTEM
            GIT_SUBMODULES_RECURSE TRUE
        )
    else()
        FetchContent_Declare(${name}
            GIT_REPOSITORY ${repo}
            GIT_TAG ${tag}
            GIT_SHALLOW TRUE
            GIT_SUBMODULES_RECURSE TRUE
        )
    endif()
    FetchContent_MakeAvailable(${name})
    _mark_fetched_content_system()
endfunction()

function(_build_archive_url out_var repo ref)
    if(repo MATCHES "github.com/")
        set(${out_var} "https://${repo}/archive/${ref}.zip" PARENT_SCOPE)
    elseif(repo MATCHES "gitlab.com/")
        string(REGEX REPLACE ".*/([^/]+)/?$" "\\1" project_name "${repo}")
        string(REPLACE "/" "-" project_name_safe "${project_name}")
        set(${out_var} "https://${repo}/-/archive/${ref}/${project_name_safe}-${ref}.zip" PARENT_SCOPE)
    else()
        set(${out_var} "https://github.com/${repo}/archive/${ref}.zip" PARENT_SCOPE)
    endif()
endfunction()

# =============================================================================
# Public API
# =============================================================================

function(fetch_dependency)
    cmake_parse_arguments(ARG
        ""
        "NAME;REPO;VERSION;COMMIT;GIT_REPOSITORY;GIT_TAG;ZIP_URL;ZIP_NAME"
        ""
        ${ARGN})

    if(NOT ARG_NAME)
        message(FATAL_ERROR "fetch_dependency: NAME is required")
    endif()

    # Git mode
    if(ARG_GIT_REPOSITORY)
        if(NOT ARG_GIT_TAG)
            message(FATAL_ERROR "fetch_dependency: GIT_REPOSITORY requires GIT_TAG")
        endif()
        _fetch_dependency_from_git("${ARG_NAME}" "${ARG_GIT_REPOSITORY}" "${ARG_GIT_TAG}")
        return()
    endif()

    # ZIP mode: build filename and URL
    if(NOT ARG_ZIP_NAME)
        if(ARG_COMMIT)
            set(ARG_ZIP_NAME "${ARG_NAME}-${ARG_COMMIT}.zip")
        elseif(ARG_VERSION)
            string(REPLACE "v" "" _version_clean "${ARG_VERSION}")
            set(ARG_ZIP_NAME "${ARG_NAME}-${_version_clean}.zip")
        else()
            message(FATAL_ERROR "fetch_dependency: Must specify VERSION, COMMIT, GIT_REPOSITORY, or ZIP_URL")
        endif()
    endif()

    if(NOT ARG_ZIP_URL)
        if(NOT ARG_REPO)
            message(FATAL_ERROR "fetch_dependency: REPO required for archive download")
        endif()

        set(_ref "${ARG_COMMIT}")
        if(NOT _ref)
            set(_ref "${ARG_VERSION}")
        endif()

        _build_archive_url(ARG_ZIP_URL "${ARG_REPO}" "${_ref}")
    endif()

    _fetch_dependency_from_zip("${ARG_NAME}" "${ARG_ZIP_URL}" "${ARG_ZIP_NAME}")
endfunction()
