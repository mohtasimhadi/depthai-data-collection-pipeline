# Installation
1. Clone the repository
```bash
git clone https://github.com/mohtasimhadi/depthai-pipeline.git
cd depthai-pipeline
```
2. Create and activate python virtual environment. *Python-3.11.0* was used during development.
```bash
python3 -m venv env
source env/bin/activate #For Linux and MacOS. Windows systems activate python virtual env in a different way!
```

3. Install dependencies
```bash
pip install -r requirements.txt
```

# Usage

```bash
python main.py -o record <output_directory>
```
