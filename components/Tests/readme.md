# criar diretório para compilação
mkdir windows_build

# Inicializar o diretorio com cmake
cd windows_build
cmake ..

# compilar
cmake --build .

# executar em
windows_build/Debug/simple_example.exe

# ou abstrair com cmake
> cmake -E make_directory build
> cmake -E chdir build cmake .. 
> cmake --build build