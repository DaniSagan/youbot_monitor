#! /usr/bin/python
# -*- coding=utf-8 -*-

import wx
from mainwindow import MainWindow

def main():
    app = wx.App()
    window = MainWindow()
    app.MainLoop()

if __name__ == "__main__": main()
