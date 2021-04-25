"""Physax setup."""

from setuptools import setup

setup(
    name="pyrigidbody3d",
    version="0.3",
    description="Very simple toy implementation of a 3d rigid body physics engine in Python.",
    long_description="Very simple toy implementation of a 3d rigid body physics engine in Python.",
    url="https://github.com/erwincoumans/py_rigidbody_3d",
    author="Erwin Coumans",
    author_email="erwincoumans@google.com",
    classifiers=[],
    packages=[
        "pyrigidbody3d"
    ],
    install_requires=[
        "numpy",
    ],
)
