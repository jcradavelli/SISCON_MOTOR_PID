
file(READ ${VERSION_GIT_TAG_FILE} VERSION_GIT_TAG)
string(STRIP "${VERSION_GIT_TAG}" VERSION_GIT_TAG)

# Verifica se a variável foi obtida corretamente
if(NOT VERSION_GIT_TAG)
    set(VERSION_GIT_TAG "undefined")
endif()

# Adiciona a definição de compilação
add_compile_definitions(
    VERSION_GIT_TAG="${VERSION_GIT_TAG}"
)
