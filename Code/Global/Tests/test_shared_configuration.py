"""Verify shared imports and model measurements without running controllers."""
import ast
from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

CODE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(CODE))
from Global import physical_parameters as parameters


class SharedConfigurationTests(unittest.TestCase):
    def measurements(self, name):
        tree = ast.parse((CODE / name).read_text())
        wanted = {'length', 'lval', 'Ival', 'm1val', 'T_dragVal', 'mcartVal',
                  'B_cart_dragVal', 'gval', 'Ts', 'length2', 'mtotal2', 'lval2', 'Ival2'}
        nodes = [node for node in tree.body if
                 isinstance(node, ast.ImportFrom) and node.module == 'Global.physical_parameters'
                 or isinstance(node, ast.Assign) and any(
                     isinstance(target, ast.Name) and target.id in wanted for target in node.targets)]
        env = {}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), name, 'exec'), env)
        return {key: env[key] for key in wanted if key in env}

    def test_existing_single_and_double_measurements_preserved(self):
        expected = dict(length=0.2, lval=0.05, Ival=0.003, m1val=0.260,
                        T_dragVal=0.0, mcartVal=0.969, B_cart_dragVal=0.5,
                        gval=9.81, Ts=0.01)
        for script in ('PythonMaster_auto.py', 'PythonMasterSwingUpExp.py'):
            self.assertEqual(self.measurements('SinglePendulum/' + script), expected)
        self.assertEqual(self.measurements('DoublePendulum/PythonMasterMath2.py'),
                         dict(length=0.2, lval=0.087, Ival=0.00123324992, m1val=0.207,
                              length2=0.25, mtotal2=0.210, lval2=0.106, Ival2=0.00197563667,
                              mcartVal=0.969 - 0.059, B_cart_dragVal=0.0, gval=9.81,
                              T_dragVal=0.0, Ts=0.01))

    def test_configuration_edits_reach_model_consumers(self):
        with patch.object(parameters, 'SINGLE_ARM', replace(parameters.SINGLE_ARM, mass_kg=0.4)):
            for script in ('PythonMaster_auto.py', 'PythonMasterSwingUpExp.py'):
                self.assertEqual(self.measurements('SinglePendulum/' + script)['m1val'], 0.4)
        with patch.object(parameters, 'DOUBLE_ARM_2', replace(parameters.DOUBLE_ARM_2, mass_kg=0.3)):
            self.assertEqual(self.measurements('DoublePendulum/PythonMasterMath2.py')['mtotal2'], 0.3)
        self.assertAlmostEqual(parameters.SINGLE_ARM.weight_n, 0.260 * 9.81)

    def test_shared_imports_from_unrelated_working_directory(self):
        # Execute the actual bootstrap/import statements in a clean process.
        names = ('SinglePendulum/PythonMaster_auto.py', 'SinglePendulum/PythonMasterSwingUpExp.py',
                 'DoublePendulum/PythonMaster2_auto.py', 'DoublePendulum/PythonMasterMath2.py',
                 'Global/AngleSensorDiagnostic.py')
        for name in names:
            path = CODE / name
            tree = ast.parse(path.read_text())
            nodes = [node for node in tree.body if
                     isinstance(node, ast.Import) and all(alias.name == 'sys' for alias in node.names)
                     or isinstance(node, ast.ImportFrom) and (node.module == 'pathlib' or node.module.startswith('Global'))
                     or isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'CODE_DIRECTORY' for t in node.targets)
                     or isinstance(node, ast.If) and 'sys.path' in ast.unparse(node.test)]
            source = ast.unparse(ast.Module(body=nodes, type_ignores=[]))
            with self.subTest(script=name), tempfile.TemporaryDirectory() as cwd:
                subprocess.run([sys.executable, '-c', f'__file__ = {str(path)!r}\n' + source],
                               cwd=cwd, check=True, capture_output=True, text=True)

    def test_helpers_have_one_canonical_location(self):
        for name in ('run_csv_logger.py', 'angle_protocol.py', 'angle_packet_fallback.py',
                     'live_offset_keys.py', 'AngleSensorDiagnostic.py'):
            self.assertTrue((CODE / 'Global' / name).is_file())
            for folder in ('SinglePendulum', 'DoublePendulum'):
                self.assertFalse((CODE / folder / name).exists())


if __name__ == '__main__':
    unittest.main()
