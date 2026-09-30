import os

from ament_pep257.main import main
import pytest


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rc = main(argv=[pkg_dir])
    assert rc == 0, f'pep257 returned {rc}'

