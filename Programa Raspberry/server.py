# -*- coding: utf-8 -*-
"""
Created on Mon Sep 26 11:14:06 2016

@author: jonathan
"""

import SimpleHTTPServer
import SocketServer



PORT = 8000
Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
httpd = SocketServer.TCPServer(("", PORT), Handler)
print "serving at port", PORT
httpd.serve_forever()

#%%
