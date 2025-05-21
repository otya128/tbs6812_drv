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

### 仕様

`DELIVERY_SYSTEM = ISDBS`かつ`STREAM_ID`が8未満のときISDB-Sの相対TS番号で選局されます。

`DELIVERY_SYSTEM = ISDBS`かつ`STREAM_ID`の上位4ビットが0xbまたは0xcのときISDB-S3のストリームIDで選局、それ以外のときISDB-SのTSIDで選局されます。
