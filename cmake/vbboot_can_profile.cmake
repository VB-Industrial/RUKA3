# Keep robot-specific FDCAN settings in RUKA3, outside the VBBoot submodule.
# CMAKE_PROJECT_VBBoot_INCLUDE runs before VBBoot creates its target, so replace
# only its transport initializer at the end of the directory evaluation.
function(ruka3_vbboot_can_profile)
    get_target_property(sources VBBoot SOURCES)
    list(FILTER sources EXCLUDE REGEX "(^|/)App/state_manager\\.c$")
    set_property(TARGET VBBoot PROPERTY SOURCES "${sources}")
    target_sources(VBBoot PRIVATE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../Boot/fdcan.c")
endfunction()
cmake_language(DEFER CALL ruka3_vbboot_can_profile)
