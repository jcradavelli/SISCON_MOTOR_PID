idf_component_register(SRCS "Version_tools.c"
                       REQUIRES
                       INCLUDE_DIRS "." 
                      )



# Obtenha a tag mais recente do Git
execute_process(
    COMMAND git describe --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Verifique se a variável foi obtida corretamente
if(NOT GIT_TAG)
    set(GIT_TAG "undefined")
endif()

# Função para adicionar a definição de compilação a todas as targets
function(add_git_tag_to_all_targets)
    get_property(targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target IN LISTS targets)
        if (TARGET ${target})
            target_compile_definitions(${target} PRIVATE GIT_TAG="${GIT_TAG}")
        endif()
    endforeach()
endfunction()

# Adicionar a definição de compilação a todas as targets
add_git_tag_to_all_targets()

