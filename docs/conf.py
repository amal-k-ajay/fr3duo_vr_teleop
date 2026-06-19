"""Sphinx configuration for the FR3 Duo Quest Teleop docs."""

import os
import sys

sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'FR3 Duo Quest Teleop'
copyright = '2026, Amal Kaithavalappil Ajay'
author = 'Amal Kaithavalappil Ajay'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
]

autodoc_mock_imports = [
    'control_msgs',
    'franka_msgs',
    'geometry_msgs',
    'rclpy',
    'scipy',
    'sensor_msgs',
    'std_srvs',
    'tf2_ros',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
