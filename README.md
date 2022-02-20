# Matric Screen

## 1. Create Project Step
### 1.1. Create main
```shell
$ idf.py create-project MatricScreen 
```

### 1.2. Create HDC1080 Sensor

```shell
$ idf.py create-component -C components/sensor HDC1080 
```

### 1.3. Create I2C Module

```shell
$ idf.py create-component -C components i2c_module
```

## 2. Build Project
### 2.1. Set the Target Chip
```shell
$ idf.py set-target esp32
```



