import os

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rc, errors = main_with_errors(argv=[pkg_dir])
    assert rc == 0, f'Found {len(errors)} flake8 errors'

