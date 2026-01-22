isort .
yapf --style .style.yapf --in-place --recursive . --exclude .direnv/ --exclude .venv
flake8 --exclude .venv --ignore E111,E114,E501,E126,W504,E125
