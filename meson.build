project('Deblock', 'cpp',
  default_options: ['buildtype=release', 'warning_level=2', 'b_lto=true', 'b_ndebug=if-release', 'cpp_std=c++17'],
  license: 'GPL-2.0-or-later',
  meson_version: '>=0.51.0',
  version: '7.1',
)

cxx = meson.get_compiler('cpp')

gcc_syntax = cxx.get_argument_syntax() == 'gcc'

if get_option('buildtype') == 'release'
  add_project_arguments(gcc_syntax ? ['-fno-math-errno', '-fno-trapping-math'] : '/GS-', language: 'cpp')
endif

sources = files('Deblock/Deblock.cpp')

if gcc_syntax
  vapoursynth_dep = dependency('vapoursynth', version: '>=55').partial_dependency(compile_args: true, includes: true)
  install_dir = vapoursynth_dep.get_variable(pkgconfig: 'libdir') / 'vapoursynth'
else
  vapoursynth_dep = []
  install_dir = get_option('libdir') / 'vapoursynth'
endif

shared_module('deblock',
  sources,
  dependencies: vapoursynth_dep,
  gnu_symbol_visibility: 'hidden',
  install: true,
  install_dir: install_dir,
)
