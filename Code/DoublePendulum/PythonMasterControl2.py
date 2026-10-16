"""Start double-pendulum balancing with receiver protocol v1.

Both supported entry points use the same calibrated controller. Configure
manual angle offsets and logging in PythonMaster2_auto.py.
The old six-byte reader and fixed encoder references are no longer used.
"""

from PythonMaster2_auto import main


if __name__ == "__main__":
    main()
