fpga_udp
========

A project modified from [example](https://github.com/pybind/python_example/) built with [pybind11](https://github.com/pybind/pybind11), 用于通过C语言编写的socket代码从网口接收高速的原始数据。This requires Python 3.7+.

Installation
------------

 - clone this repository
 - `pip install ./fpga_udp`

CI Examples
-----------

There are examples for CI in `.github/workflows`. A simple way to produces
binary "wheels" for all platforms is illustrated in the "wheels.yml" file,
using [`cibuildwheel`][]. You can also see a basic recipe for building and
testing in `pip.yml`, and `conda.yml` has an example of a conda recipe build.


Building the documentation
--------------------------

Documentation for the example project is generated using Sphinx. Sphinx has the
ability to automatically inspect the signatures and documentation strings in
the extension module to generate beautiful documentation in a variety formats.
The following command generates HTML-based reference documentation; for other
formats please refer to the Sphinx manual:

 - `cd fpga_udp/docs`
 - `make html`

License
-------

pybind11 is provided under a BSD-style license that can be found in the LICENSE
file. By using, distributing, or contributing to this project, you agree to the
terms and conditions of this license.

Test call
---------

```python
import fpga_udp
print(fpga_udp.add(1, 2))
print(fpga_udp.getOverflowCnt())
```

[`cibuildwheel`]:          https://cibuildwheel.readthedocs.io
