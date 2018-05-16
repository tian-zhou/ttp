#!/usr/bin/env python

from tkinter import *
# from tkinter import scrolledtext

class GUI:
    def __init__(self):
        self.window = Tk()
        self.window.title("Turn-taking")
        self.window.geometry('600x800') # width x height

    def clicked(self):
        msg = "Welcome to " + self.txt.get()
        self.lab.configure(text=msg)
    
    def run(self):
        # Label
        self.lab = Label(self.window, text="Hello", font=("Arial Bold", 16), padx=10, pady=10)
        self.lab.grid(column=0, row=0)
        
        # Button
        self.but = Button(self.window, text="Click Me", command=self.clicked, padx=10, pady=10) #, bg="orange", fg="red")
        self.but.grid(column=1, row=0)

        # text Entry
        self.txt = Entry(self.window,width=10)
        self.txt.grid(column=0, row=1)
        # self.txt.focus() # set focus so that we can directly enter

        # ScrolledText
        # self.stxt = scrolledtext.ScrolledText(self.window,width=40,height=10)
        # self.stxt.grid(column=0, row=2)

        self.window.mainloop()

def main():
    gui = GUI()
    gui.run()

if __name__ == '__main__':
    main()
