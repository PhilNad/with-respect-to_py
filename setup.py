from setuptools import setup

setup(
    name='with-respect-to',
    version='0.1.0',
    description='Simple library that manages databases of 3D transformations with explicit accessors.',
    author='Philippe Nadeau',
    author_email='philippe.nadeau@robotics.utias.utoronto.ca',
    license='MIT',
    packages=['WRT'],
    install_requires=['scipy', 'numpy', 'sympy','spatialmath-python','matplotlib'],
    include_package_data=False
)