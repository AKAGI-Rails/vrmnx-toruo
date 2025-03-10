# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../'))

import toruo


# -- Project information -----------------------------------------------------

project = '撮る夫くん'
copyright = '2021-2025, AKAGI Rails'
author = 'AKAGI Rails'

# The full version, including alpha/beta/rc tags
release = toruo.__version__


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    #'sphinx.ext.autosummary',
    #'sphinx_automodapi.automodapi',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
    'sphinx.ext.todo',
    'sphinx.ext.githubpages',
    'myst_parser',
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# MyST Parserの設定（数式サポート）
myst_enable_extensions = [
    "dollarmath",  # $$形式の数式
    "amsmath",     # \begin{align}などの高度な数式
]

napoleon_use_rtype = False
todo_include_todos = True

# Add any paths that contain templates here, relative to this directory.
#templates_path = ['_templates']

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'ja'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_material'

# Material theme options (see theme.conf for more information)
html_theme_options = {

    # Set the name of the project to appear in the navigation.
    'nav_title': '撮る夫くん',

    # Specify a base_url used to generate sitemap.xml. If not
    # specified, then no sitemap will be built.
    'base_url': 'https://akagi-rails.github.io/vrmnx-toruo',

    # Set the color and the accent color
    'color_primary': 'teal',
    'color_accent': 'green',
    
    'logo_icon': 'videocam',

    # Set the repo location to get a badge with stats
    'repo_url': 'https://github.com/AKAGI-Rails/vrmnx-toruo',
    'repo_name': '撮る夫くん',

    # Visible levels of the global TOC; -1 means unlimited
    'globaltoc_depth': 3,
    # If False, expand all TOC entries
    'globaltoc_collapse': False,
    # If True, show hidden TOC entries
    'globaltoc_includehidden': False,
}

html_sidebars = {
    "**": ["logo-text.html", "globaltoc.html", "localtoc.html", "searchbox.html"]
}
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
#html_static_path = ['_static']

# html_context（静的値のみ）
html_context = {
    "project": "撮る夫くん",
    "version": release,
}

# MathJax設定
mathjax_path = 'https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js'
