import wx
import matplotlib
matplotlib.use('WX')
import matplotlib.figure 
import matplotlib.backends.backend_wxagg 
import pylab



class HistoryPlotFrame(wx.Frame):
    def __init__(self, parent, history):
        wx.Frame.__init__(self, parent, -1, history.getTitle())
        self.Bind(wx.EVT_CLOSE, self.onClose)

        dpi = 100
        rc = matplotlib.figure.SubplotParams(left=0.125, bottom=0.12, right=0.99, top=0.99, wspace=0.001, hspace=0.1)
        fig = matplotlib.figure.Figure((3.0, 3.0), dpi=dpi, subplotpars=rc)

        fp = matplotlib.font_manager.FontProperties(size=10)

        axes = fig.add_subplot(111)
        axes.set_axis_bgcolor('white')
        #pylab.setp(axes.get_xticklabels(), fontsize=6)
        #pylab.setp(axes.get_yticklabels(), fontsize=8)

        tsd_list = history.getAllTimestepData()
        time_list = []
        dropped_list = []
        late_list = []
        for tsd in tsd_list:
            time_list.append(tsd.getTimestamp().to_sec())
            dropped_list.append(tsd.system.master.dropped)
            late_list.append(tsd.system.master.late)

        start_time=tsd_list[0].getTimestamp().to_sec()
        time_list = (pylab.array(time_list) - start_time) / 60.
        dropped_list = pylab.array(dropped_list)
        late_list = pylab.array(late_list)

        axes.plot(time_list, dropped_list, 'r-', linewidth=1, picker=5, label='dropped')
        axes.plot(time_list, late_list, 'b-', linewidth=1, picker=5, label='late')
        axes.legend(loc='best', prop=fp)        
        axes.xaxis.set_label_text('Time (minutes)')
        axes.yaxis.set_label_text('Packets')
        #print dir(axes.yaxis)
        print axes.yaxis.get_axes()
        axes.grid(True, color='gray')

        canvas = matplotlib.backends.backend_wxagg.FigureCanvasWxAgg(self, -1, fig)
        #canvas.mpl_connect('pick_event', self.onPick)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(sizer)

        
    def onClose(self, event):
        self.Destroy()

