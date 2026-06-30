project('Deblock', 'cpp',
    default_options: ['buildtype=release', 'warning_level=2', 'b_lto=true', 'b_ndebug=if-release', 'cpp_std=c++17'],
    license: 'GPL-2.0-or-later',
    license_files: 'LICENSE',
    meson_version: '>=1.2.3',
    version: '8.1',
)

py = import('python').find_installation(pure: false)

incdir = include_directories(run_command('vapoursynth', 'get-include', check: true).stdout().strip())

libs = []

if host_machine.cpu_family().startswith('x86')
    add_project_arguments('-DDEBLOCK_X86', language: 'cpp')

    libs += static_library('instrset_detect',
        files('src/vectorclass/instrset_detect.cpp'),
        gnu_symbol_visibility: 'hidden',
    )

    libs += static_library('sse4',
        files('src/deblock_sse4.cpp'),
        cpp_args: '-march=x86-64-v2',
        gnu_symbol_visibility: 'hidden',
        include_directories: incdir,
    )
endif

shared_module('deblock',
    files('src/deblock.cpp'),
    gnu_symbol_visibility: 'hidden',
    include_directories: incdir,
    install: true,
    install_dir: py.get_install_dir() / 'vapoursynth/plugins',
    link_with: libs,
    name_prefix: '',
)
