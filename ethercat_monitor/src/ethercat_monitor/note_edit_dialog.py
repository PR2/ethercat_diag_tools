
import wx

class NoteEditDialog(wx.Dialog):
    def __init__(self, parent, note):
        wx.Dialog.__init__(self,parent,-1,title='Edit Note')

        self.note = note

        self.note_textctrl = wx.TextCtrl(self)
        self.note_textctrl.SetToolTip(wx.ToolTip("Change note messsage"))
        self.note_textctrl.SetValue(note.note_msg)

        self.cancel_button = wx.Button(self, -1, "Cancel")
        self.Bind(wx.EVT_BUTTON, self.onCancel, self.cancel_button)        

        self.ok_button = wx.Button(self, -1, "OK")
        self.Bind(wx.EVT_BUTTON, self.onOk, self.ok_button)        

        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        hsizer.Add(self.cancel_button, 0)
        hsizer.Add(self.ok_button, 0)

        vsizer = wx.BoxSizer(wx.VERTICAL)
        vsizer.Add(self.note_textctrl, 0, wx.EXPAND)
        vsizer.Add(hsizer, 0, wx.EXPAND)
        
        self.SetSizer(vsizer)
        self.Centre()
        self.Layout()
        self.Show(True) 

    def onCancel(self, event):
        self.Close()

    def onOk(self, event):
        self.note.note_msg = str(self.note_textctrl.GetValue())
        self.Close()

    def onClose(self, event):
        self.Close()
