from ament_pep257.main import main


def test_pep257():
    assert main(argv=['.', '--ignore=D100,D101,D102,D103,D104,D105,D107']) == 0
