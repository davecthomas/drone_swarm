# Configure compiler warnings across supported toolchains.
function(enable_project_warnings target_name)
    if (MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive- /EHsc /external:anglebrackets /external:W0)
    else()
        target_compile_options(${target_name} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wsign-conversion
            -Wshadow
            -Wnon-virtual-dtor
            -Wold-style-cast
            -Woverloaded-virtual
            -Wdouble-promotion
            -Wformat=2
        )
    endif()
endfunction()
