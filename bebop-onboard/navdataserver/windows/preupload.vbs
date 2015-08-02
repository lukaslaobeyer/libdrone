set tshell = CreateObject("WScript.Shell")
tshell.run("Telnet")
WScript.Sleep 500
tshell.SendKeys("Open 192.168.42.1 23")
WScript.Sleep 500
tshell.SendKeys("{Enter}")
WScript.Sleep 500
tshell.SendKeys("mkdir /data/ftp/internal_000/navdataserver")
WScript.Sleep 500
tshell.SendKeys("{Enter}")
WScript.Sleep 500
tshell.SendKeys("exit")
WScript.Sleep 500
tshell.SendKeys("{Enter}")
WScript.Quit 0