from setuptools import find_packages, setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(name='PACKAGE_NAME__PASCAL',
      version='PACKAGE_VERSION',
      author='AUTHOR_NAME',
      author_email='AUTHOR_EMAIL',
      url='https://github.com/best-of-acrv/refinenet',
      description='RefineNet semantic image segmentation',
      license_files=['LICENSE_FILE'],
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages=find_packages(),
      install_requires=PACKAGE_DEPENDENCIES,
      TEMPLATE_START ADD_ENTRYPOINT
      entry_points={'console_scripts': ['ADD_ENTRYPOINT=PACKAGE_NAME__PASCAL.__main__:main']},
      TEMPLATE_END
      classifiers=(
          "Development Status :: 4 - Beta",
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: BSD License",
          "Operating System :: OS Independent",
      ))
