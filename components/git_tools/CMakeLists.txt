idf_component_register(SRCS "git_tools.c"
                       REQUIRES
                       INCLUDE_DIRS "." 
                      )



# Obtenha a tag mais recente do Git
execute_process(
    COMMAND git describe --tags --dirty --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE __GIT_DESCRIBE_TAGS_DIRTY_ALWAYS__
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Verifique se a variável foi obtida corretamente
if(NOT __GIT_DESCRIBE_TAGS_DIRTY_ALWAYS__)
    set(__GIT_DESCRIBE_TAGS_DIRTY_ALWAYS__ "undefined")
endif()

# Função para adicionar a definição de compilação a todas as targets
function(add___GIT_DESCRIBE_TAGS_DIRTY_ALWAYS___to_all_targets)
    get_property(targets DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target IN LISTS targets)
        if (TARGET ${target})
            target_compile_definitions(${target} PRIVATE __GIT_DESCRIBE_TAGS_DIRTY_ALWAYS__="${__GIT_DESCRIBE_TAGS_DIRTY_ALWAYS__}")
        endif()
    endforeach()
endfunction()

# Adicionar a definição de compilação a todas as targets
add___GIT_DESCRIBE_TAGS_DIRTY_ALWAYS___to_all_targets()

