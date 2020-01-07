# FRC2020

## Dependencies
Dependencies are located in `Pipfile`

Get `pipenv` via `pip`

```bash
python -m pip install --user pipenv
```

Then install dependencies

```bash
pipenv install
```

If you need to install more dependencies into the environment a) tell me and b) use the following command

```bash
pipenv install package_name
```

Make sure you have [cue](https://github.com/cuelang/cue) installed and in your `$PATH` as well to build and manage data/config files

## Running/deploying/whatever

Once you have dependencies installed `cd` into the folder containing the robot class and run something along the lines of

```bash
./build_linux.sh && pipenv run python robot.py test
```

Now obviously you want to use `build_windows.bat` if you're on Windows or `deploy` if you want to deploy the code as opposed to running tests but that's the gist of the command