@echo on

:: Windows 10 (as of recently) comes with cURL
curl -L https://zenodo.org/record/7662056/files/DatasetMotionsProMP.zip -o dataset.zip
:: Windows 10 also comes with tar, although slightly iffy if you should use it with .zip. It seems to work fine, though.
tar -xf dataset.zip --strip-components 1
:: delete it afterwards
del dataset.zip