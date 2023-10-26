
 # 查看局域网内已经连接的ip地址
 nmap -sP 192.168.2.2-255|grep -i 'report for'|awk '{print $5;}'|cat -n
