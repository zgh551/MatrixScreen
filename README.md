# Matric Screen

## create step
1. create main
```shell
$ idf.py create-project MatricScreen 
```

2. create hdc1080 sensor

```shell
$ idf.py create-component -C components/sensor HDC1080 
```

3. create i2c module

```shell
$ idf.py create-component -C components i2c_module
```
