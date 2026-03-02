# Donanım Testi Hız Sınırı — Değişiklik Raporu

**Amaç:** İlk donanım testinde güvenlik için tüm motorların maksimum hızını %20 ile sınırlamak.  
**Sabit:** `Constants.ControllerConstants.TEST_SPEED_MULTIPLIER = 0.2`

Test bittikten sonra tam hıza dönmek için: `Constants.java` içinde `TEST_SPEED_MULTIPLIER` değerini `1.0` yapmanız yeterlidir.

---

## 1. Constants.java

- **Satır / bölge:** `ControllerConstants` sınıfı içi (DEADBAND tanımından sonra).
- **Yapılan:** Yeni sabit eklendi:
  - `TEST_SPEED_MULTIPLIER = 0.2` (açıklama: donanım testi için genel hız çarpanı; 1.0 = tam hız).

---

## 2. DriverControls.java (Joystick — 2 kumanda)

- **Import:** `import frc.robot.Constants;` eklendi (TEST_SPEED_MULTIPLIER erişimi için).
- **curvatureDriveCommand — throttle (ileri/geri):**
  - Önce: `return input * 0.5;`
  - Sonra: `return input * 0.5 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER;`
  - Böylece sürüş throttle’ı (controller + controller2 sol stick Y) %20 ile sınırlandı.
- **curvatureDriveCommand — turn (dönüş):**
  - Önce: `return input * 0.5;`
  - Sonra: `return input * 0.5 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER;`
  - Böylece dönüş (sağ stick X, her iki kumanda) %20 ile sınırlandı.
- **Değiştirilmeyen:** Kumanda portları, deadband, quick-turn koşulu, buton atamaları ve komut yapısı aynı kaldı.

---

## 3. TurretSubsystem.java

- **Metot:** `set(double dutyCycle)`
- **Yapılan:** `turret.set(dutyCycle)` → `turret.set(dutyCycle * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Not:** `Superstructure.stopAllCommand()` içinde `turret.set(0)` çağrısı 0 ile çarpıldığı için davranış değişmez (yine 0).

---

## 4. KickerSubsystem.java

- **Metot:** `feedCommand()`
- **Yapılan:** `kicker.set(KICKER_SPEED)` → `kicker.set(KICKER_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Değiştirilmeyen:** `stopCommand()` (zaten `kicker.set(0)`).

---

## 5. IntakeSubsystem.java

- **Metot:** `intakeCommand()`
  - `intake.set(INTAKE_SPEED)` → `intake.set(INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Metot:** `ejectCommand()`
  - `intake.set(-INTAKE_SPEED)` → `intake.set(-INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Metot:** `deployAndRollCommand()` (run içinde)
  - `smc.setDutyCycle(INTAKE_SPEED)` → `smc.setDutyCycle(INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Metot:** `backFeedAndRollCommand()` (run içinde)
  - `smc.setDutyCycle(-INTAKE_SPEED)` → `smc.setDutyCycle(-INTAKE_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Değiştirilmeyen:** Pivot komutları, rezero, diğer metot isimleri ve yapı.

---

## 6. HopperSubsystem.java

- **Metot:** `feedCommand()`
  - `hopper.set(HOPPER_SPEED)` → `hopper.set(HOPPER_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Metot:** `backFeedCommand()`
  - `hopper.set(-HOPPER_SPEED)` → `hopper.set(-HOPPER_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Metot:** `reverseCommand()`
  - `hopper.set(-HOPPER_SPEED)` → `hopper.set(-HOPPER_SPEED * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER)`
- **Değiştirilmeyen:** `stopCommand()` (zaten `hopper.set(0)`).

---

## 7. StandaloneTestTurretSubsystem.java

- **Import:** `import frc.robot.Constants;` eklendi.
- **Metot:** `manualDriveCommand(int controllerPort)` — run() bloğu içinde:
  - Turret: `turretMotor.set(TURRET_SPEED)` / `turretMotor.set(-TURRET_SPEED)` → aynı değerler `* Constants.ControllerConstants.TEST_SPEED_MULTIPLIER` ile çarpıldı.
  - Shooter: `shooterLeader.set(SHOOTER_SPEED)` → `SHOOTER_SPEED * TEST_SPEED_MULTIPLIER`.
  - Feed: `hopperMotor.set(FEED_SPEED)` ve `kickerMotor.set(FEED_SPEED)` → `FEED_SPEED * TEST_SPEED_MULTIPLIER`.
  - Sıfırlama (else / finallyDo): `set(0)` çağrıları olduğu gibi bırakıldı.
- **Değiştirilmeyen:** CAN ID’ler, motor konfigürasyonları, buton eşleştirmeleri ve komut isimleri.

---

## 8. ShooterSubsystem.java

- **Metot:** `spinUp()`
- **Yapılan:** Hedef hız sınırlandı: `setSpeed(RPM.of(5500))` → `setSpeed(RPM.of(5500 * Constants.ControllerConstants.TEST_SPEED_MULTIPLIER))` (testte ~1100 RPM).
- **Değiştirilmeyen:** `stop()` (zaten 0 RPM), `setSpeed` / `setSpeedDynamic` imzaları ve diğer mantık.

---

## Özet Tablo

| Dosya | Değişen metotlar / satırlar | Ne yapıldı |
|-------|----------------------------|------------|
| Constants.java | ControllerConstants | TEST_SPEED_MULTIPLIER = 0.2 eklendi |
| DriverControls.java | throttle/turn lambdaları | Joystick çıktıları TEST_SPEED_MULTIPLIER ile çarpıldı |
| TurretSubsystem.java | set(double) | dutyCycle çarpan ile çarpılıyor |
| KickerSubsystem.java | feedCommand() | KICKER_SPEED çarpan ile çarpılıyor |
| IntakeSubsystem.java | intakeCommand, ejectCommand, deployAndRollCommand, backFeedAndRollCommand | Tüm ilgili set/setDutyCycle değerleri çarpan ile çarpıldı |
| HopperSubsystem.java | feedCommand, backFeedCommand, reverseCommand | HOPPER_SPEED çarpan ile çarpıldı |
| StandaloneTestTurretSubsystem.java | manualDriveCommand run() | Turret, shooter, hopper, kicker set değerleri çarpan ile çarpıldı |
| ShooterSubsystem.java | spinUp() | Hedef RPM (5500) çarpan ile çarpıldı |

---

## Uyulunan kurallar

- **CAN ID:** Hiçbir yerde değiştirilmedi veya silinmedi.
- **Subsystem/Command yapısı:** Mimari, sınıf/metot isimleri ve temel mantık korundu; yalnızca sayısal çıktılar çarpan ile sınırlandı.
- **Değişiklikler minimal:** Sadece hız/hız çarpanı ile ilgili satırlar güncellendi; test sonrası tam hız için sadece `TEST_SPEED_MULTIPLIER = 1.0` yeterli.
