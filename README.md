# F1Tenth

## Run code

If you have pretrained model just go to `evaluation`

### Prepare dataset

1. Download ros bags
2. Use `Preprocess_Bag.ipynb` to export images frmo bags
3. Create list of files:
```
cd DIRECTORY_WITH_EXPORTED_IMAGES
ls -1 > images_list.csv
```

### Training

```
python model.py --list images_list.csv --output logs --dataset dataset001/images/
```

### Evaluation

```
python evaluate.py --list images_list.csv --output logs --dataset dataset001/images/ --model ./logs/logs_1529490181/weights.0040-0.036.hdf5
```

## Datasets

Go to:
[Datasets](https://github.com/karolmajek/f1tenth/blob/master/Datasets.md)

## Models

Go to:
[Models](https://github.com/karolmajek/f1tenth/blob/master/Models.md)

## Team members
- [Karol Majek](https://karolmajek.pl)
- [Maciej Dziubiński](https://www.linkedin.com/in/maciej-dziubinski/)
- [Łukasz Sztyber](https://www.linkedin.com/in/lukaszsztyber/)

## More info
- [Deep Drive PL Blog](https://deepdrive.pl)
- [F1/10](http://f1tenth.org/)
- [Blog post](https://deepdrive.pl/samochod-autonomiczny-w-skali/)
