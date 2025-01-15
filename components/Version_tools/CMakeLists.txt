idf_component_register(SRCS "Version_tools.c"
                       REQUIRES
                       INCLUDE_DIRS "."
                      )




# Define a função para obter a tag mais recente do Git
add_custom_target(get_git_tag ALL
    COMMAND ${CMAKE_COMMAND} -E echo "Obtendo a tag mais recente do Git..."
    # COMMAND git describe --tags > ${CMAKE_BINARY_DIR}/git_tag.txt
)

# Leia o conteúdo do arquivo gerado
file(READ ${CMAKE_BINARY_DIR}/git_tag.txt VERSION_GIT_TAG)
string(STRIP "${VERSION_GIT_TAG}" VERSION_GIT_TAG)

# Verifique se a variável foi obtida corretamente
if(NOT VERSION_GIT_TAG)
    set(VERSION_GIT_TAG "undefined")
endif()

# Adicione a definição de compilação
add_compile_definitions(
    VERSION_GIT_TAG="${VERSION_GIT_TAG}"
)


