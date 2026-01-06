## tbs6812_drv

ISDB-T/ISDB-S/ISDB-S3対応Linux向けTBS6812 a.k.a. PT4Kドライバ

カーネルモジュールのビルド環境を構築後、dkmsでインストールします。

```sh
sudo cp -a ./ /usr/src/tbs6812_drv-0.0.1
sudo dkms add tbs6812_drv/0.0.1
sudo dkms install tbs6812_drv/0.0.1
```

DVBデバイスとして見えるのでdvb-toolsなどを導入するとストリームを得ることができます。

```sh
dvbv5-zap -a 0 -c default-channels.conf defaultTLV -r -P -o defaultTLV.mmts
dvbv5-zap -a 0 -c default-channels.conf defaultTS -r -P -o defaultTS.ts
```

recdvbのfork <https://github.com/otya128/recdvb> を用いるとストリームIDを指定してストリームを得ることができます。

```sh
# デフォルトTLV (ISDB-S3)を受信
recdvb --dev 0 0xb110 - defaultTLV.mmts
# デフォルトTS (ISDB-S)を受信
recdvb --dev 0 0x40f1 - defaultTS.ts
# 地上波の物理26chを受信
recdvb --dev 0 26 - terr-26ch.ts
```

### 仕様

`DELIVERY_SYSTEM = ISDBS`かつ`STREAM_ID`が8未満のときISDB-Sの相対TS番号で選局されます。

`DELIVERY_SYSTEM = ISDBS`かつ`STREAM_ID`の下位16ビット部の上位4ビットが0xbまたは0xcのとき (0xbXXX, 0xcXXXX) ISDB-S3のストリームIDで選局、それ以外のときISDB-SのTSIDで選局されます。
