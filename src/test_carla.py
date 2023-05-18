import io
import sys
import os
from contextlib import redirect_stdout
import unittest

import config_utils as conf
from src.CarlaMain import CarlaMain


class TestCarla(unittest.TestCase):
    def test_configs(self):
        configs_path = r"../configs"
        configs = [elem for elem in os.listdir(configs_path) if elem.endswith('conf')]

        # Supress prints
        for config in configs:
            with io.StringIO() as buffer, redirect_stdout(buffer):
                result = conf.config_setup(f'{configs_path}/{config}')
            try:
                self.assertTrue(result)
            except AssertionError:
                print(f'Invalid config: {configs_path}/{config}')
                raise


if __name__ == '__main__':
    unittest.main()
