# Operatör Kumandası — Manuel Test Atamaları Değişiklik Raporu

**Amaç:** Alt sistemleri tek tek ve izole manuel test etmek. Tüm atamalar "basılı tutunca çalışsın, bırakınca dursun" (.whileTrue). TEST_SPEED_MULTIPLIER (%20) korundu.

---

## Değiştirilen Dosyalar

### 1. IntakeSubsystem.java

- **setPivotTarget(Angle angle):** Pivot hedef açısını ayarlar (manuel testte her döngüde çağrılır).
- **holdPivot():** Pivotu mevcut açıda tutar (buton bırakıldığında çağrılır).

Bu iki public metot, manuel test için pivot yukarı/aşağı RunCommand’lerinde kullanılıyor.

---

### 2. OperatorControls.java

- **Import:** `edu.wpi.first.wpilibj2.command.Commands` eklendi.
- **Yeni atamalar (mevcut atamalara ek olarak):**

| Buton | Alt sistem | Komut / davranış |
|------|------------|-------------------|
| **D-Pad Yukarı (POV 0)** | Intake Pivot | Yukarı (0°) — whileTrue: RunCommand setPivotTarget(0°), finallyDo holdPivot |
| **D-Pad Aşağı (POV 180)** | Intake Pivot | Aşağı (148°) — whileTrue: RunCommand setPivotTarget(148°), finallyDo holdPivot |
| **Right Bumper (RB)** | Intake Rollers | whileTrue: intake.intakeCommand() (TEST_SPEED_MULTIPLIER uygulanıyor) |
| **X** | Kicker | whileTrue: kicker.feedCommand() (TEST_SPEED_MULTIPLIER uygulanıyor) |
| **B** | Shooter | whileTrue: spinUp → waitForever, finallyDo shooter.stop().schedule() (TEST_SPEED_MULTIPLIER spinUp içinde) |

**Not:** X ve B’de önceden de atama vardı (X = stop shooting, B = back feed). Şu an aynı tuşa hem eski hem yeni atama bağlı; basılı tutulunca her iki komut da çalışır. Sadece manuel test istiyorsanız eski X/B satırlarını yoruma alabilirsiniz.

---

## TEST_SPEED_MULTIPLIER

- Silinmedi, atlanmadı.
- intakeCommand(), feedCommand(), spinUp() zaten Constants.ControllerConstants.TEST_SPEED_MULTIPLIER kullanıyor; manuel test komutları bu komutları kullandığı için %20 hız sınırı geçerli.
- Intake pivot pozisyon kontrollü (açı hedefi); hız sınırı pivot PID/feedforward tarafından belirleniyor.

---

## Bırakınca Durma

- Tüm yeni atamalar `.whileTrue(...)` ile yapıldı; buton bırakılınca komut iptal olur.
- Intake pivot: `finallyDo(superstructure.intake::holdPivot)` ile bırakınca mevcut açıda tutulur.
- Intake/Kicker: `intakeCommand()` / `feedCommand()` zaten `finallyDo(() -> smc.setDutyCycle(0))` içeriyor; bırakınca çıkış 0’a çekilir.
- Shooter: `finallyDo(() -> superstructure.shooter.stop().schedule())` ile bırakınca stop komutu çalışır.

---

## Operatör Kumandası — Tüm Tuşların Görevleri

| Buton | Davranış |
|-------|----------|
| **Start** | Bir kez: Intake pivot + turret rezero (rezeroIntakePivotAndTurretCommand). |
| **Right Trigger (RT)** | Basılı tutunca: Turret açı +0.1° (addAngle). |
| **Left Trigger (LT)** | Basılı tutunca: Turret açı -0.1° (addAngle). |
| **Left Bumper (LB)** | Basılı: Intake deploy + roller (setIntakeDeployAndRoll). Bırakınca: Intake yukarı topla (retractIntakeUpCommand). |
| **Y** | Bir kez: Atış (shootCommand — shooter spinUp). |
| **X** | Basılı: (1) Shooter durdur (stopShootingCommand). (2) Manuel test: Kicker ileri (feedCommand). |
| **A** | Basılı: Hepsi besleme (feedAll — hopper + kicker). Bırakınca stopFeedingAll. |
| **B** | Basılı: (1) Geri besleme (backFeedAll — hopper + intake geri). (2) Manuel test: Shooter spinUp (bırakınca stop). |
| **D-Pad Yukarı (POV 0)** | Bir kez: Turret ileri (0°). Basılı: Manuel test — Intake pivot yukarı (0°). |
| **D-Pad Sol (POV 270)** | Bir kez: Turret sol (45°). |
| **D-Pad Sağ (POV 90)** | Bir kez: Turret sağ (-45°). |
| **D-Pad Aşağı (POV 180)** | Basılı: Manuel test — Intake pivot aşağı (148°). |
| **Right Bumper (RB)** | Toggle: Hareket halinde nişan (ShootOnTheMoveCommand). Basılı: Manuel test — Intake roller ileri. |
