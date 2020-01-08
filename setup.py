
import setuptools

setuptools.setup(
    name="melopero_lsm9ds1",
    version="0.1.0",
    description="A module to easily access Melopero's LSM9DS1 sensor's features",
    url="https://github.com/melopero/Melopero_LSM9DS1",
    author="Melopero",
    author_email="info@melopero.com",
    license="MIT",
    packages=setuptools.find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.5",
    ],
    install_requires=["smbus2", "RPi.GPIO"],
)
