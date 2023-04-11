# Setup:
1. Ensure 7785_model.pkl is in the same directory as classifier.py
2. Place all test images and labels.txt inside the same dir. 

## To Train the model:
```python
$ python3 classifier.py 1 
```
## To Test the model: 
```python
$ python3 classifier.py 0
```

## To pass the test directory as an argument:
```python
$ python3 classifier.py 0 <dir_name>
```

Example:
```python
python classifier.py 0 ./2022Fimgs/
```

## Directory Structure
--classifier.py

--Readme.md

--7785_model.pkl

-- Test Directory

    -- *.png

    -- *.jpg

    -- labels.txt
