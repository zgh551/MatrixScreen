# Matric Screen

## 1. Create Project Step
### 1.1. Create main
```shell
$ idf.py create-project MatricScreen 
```

### 1.2. Create HDC1080 Sensor

```shell
$ idf.py create-component -C components hdc1080 
```

### 1.3. Create I2C Module

```shell
$ idf.py create-component -C components i2c_module
```

### 1.4. Create CCS811 Sensor
```shell
$ idf.py create-component -C components ccs811
```

## 2. Build Project
### 2.1. Set the Target Chip
```shell
$ idf.py set-target esp32
```

### 2.2. Configure Menu 
```shell
$ idf.py menuconfig
```

### 2.3. Build Project
```shell
$ idf.py build
```
