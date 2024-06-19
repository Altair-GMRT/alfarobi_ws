PORT=$(netstat -ap | grep :12392 | grep -oE '[^ ]+$' | awk -F '/' '{print $1}')
kill $PORT