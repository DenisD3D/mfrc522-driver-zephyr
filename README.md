# MFRC522 Driver for Zephyr

This library is a zephyr port of MFRC522 Arduino library: https://github.com/miguelbalboa/rfid/ 

## Usage

In `west.yml`:
```yaml
manifest:
  ...
  projects:
    ...
    - name: mfrc522
      url: https://github.com/DenisD3D/mfrc522-driver-zephyr
      revision: main
      path: modules/mfrc522

```

In `prj.conf`:
```
CONFIG_MFRC522=y
```

In `app.overlay`:
```dts
&spi3 {
	status = "okay";
	cs-gpios = <&gpiod 13 GPIO_ACTIVE_LOW>;

	mfrc522: mfrc522@0 {
		compatible = "nxp,mfrc522";
		reg = <1>;
		spi-max-frequency = <1000000>;
		reset-gpios = <&gpiob 9 GPIO_ACTIVE_HIGH>;
	};
```

In `main.c`:
```c
#include <mfrc522.h>

int main() {
	const struct device *mfrc522_dev = DEVICE_DT_GET(DT_NODELABEL(mfrc522));
	
	if (!device_is_ready(mfrc522_dev)) {
		LOG_ERR("MFRC522 initialization failed");
        	return -1;
	}


	while (1) {
		struct Uid uid = {0};
		if (mfrc522_picc_is_new_card_present(mfrc522a_dev)) {
			if (mfrc522_picc_read_card_serial(mfrc522a_dev, &uid)) {
				LOG_HEXDUMP_INF(uid.uid_byte, uid.size, "New Card, UID:");
				mfrc522_picc_halt_a(mfrc522a_dev);
			}
		}
		k_sleep(K_MSEC(100));
	}

	return 0;
}
```
