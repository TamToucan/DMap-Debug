BUILD_LIB(Stuff)

if(MSVC)
    target_compile_options(Stuff PRIVATE
        /wd4838  # conversion from 'type' to 'type' requires a narrowing conversion
    )
else()
    set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/TGA.cpp
        PROPERTIES COMPILE_FLAGS "-Wno-narrowing"
    )
endif()
