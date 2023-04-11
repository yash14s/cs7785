# Setup:
1. Ensure svm_model.pickle is in the same directory as classifier.py
2. Place all images and labels inside the same dir. 

## To Train the model:
```python
$ python3 classifier.py 1 
```
## To Test the model: 
```python
$ python3 classifier.py 0
```

## Pass the test directory and label name as an argument:
```python
$ python3 classifier.py 0 <dir_name> <labels_name>
```

Example:
Default options are DIR = "./2022Fheldout/" and labels_name="labels.txt". You can specify different arguments as shown below:
```python
python3 classifier.py 0 ./2022Fimgs/
python3 classifier.py 0 ./2022Fimgs/ test.txt
python3 classifier.py 0 ./2022Fimgs/ train.txt
python3 classifier.py 0 ./2022Fimgs/ labels.txt
python3 classifier.py 0 ./2022Fheldout/ test.txt
```


## Directory Structure
--classifier.py

--Readme.md

--svm_model.pickle

-- templates

-- Test Directory

    -- *.png


    -- labels.txt
