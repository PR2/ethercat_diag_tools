
import wx

class SerialInputDialog(wx.Dialog):
    def __init__(self, parent, serial):
        wx.Dialog.__init__(self,parent,-1,title='Enter Serial Number')

        self.serial = serial

        self.serial_textctrl = wx.TextCtrl(self)
        self.serial_textctrl.SetToolTip(wx.ToolTip("Enter component serial number here."))
        self.serial_textctrl.SetValue(self.serial)

        self.serial_label = wx.StaticText(self, label="Serial Number")

        self.cancel_button = wx.Button(self, -1, "Cancel")
        self.Bind(wx.EVT_BUTTON, self.onCancel, self.cancel_button)        

        self.ok_button = wx.Button(self, -1, "OK")
        self.Bind(wx.EVT_BUTTON, self.onOk, self.ok_button)

        hsizer1 = wx.BoxSizer(wx.HORIZONTAL)
        hsizer1.Add(self.serial_label, 0, wx.EXPAND)
        hsizer1.Add(self.serial_textctrl, 1, wx.EXPAND)

        hsizer2 = wx.BoxSizer(wx.HORIZONTAL)
        hsizer2.Add(self.cancel_button, 0, wx.EXPAND)
        hsizer2.Add(self.ok_button, 0, wx.EXPAND)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(hsizer1, 0, wx.EXPAND)
        vsizer.Add(hsizer2, 0, wx.EXPAND)
        
        self.SetSizer(vsizer)
        self.Centre()
        self.Layout()
        self.Show(True) 

    def onCancel(self, event):
        self.EndModal(wx.ID_CANCEL)
        self.Close()

    def onOk(self, event):
        self.serial = str(self.serial_textctrl.GetValue())
        self.EndModal(wx.ID_OK)
        self.Close()

    def onClose(self, event):
        self.EndModal(wx.ID_CANCEL)
        self.Close()
