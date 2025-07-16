#!/usr/bin/env python3
import os
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
import csv
import shutil
import time
import zipfile

RESULTS_DIR = "/home/ros/eyegaze_ws/results"

class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Participant Dashboard")
        self.geometry("800x400")
        self.resizable(False, False)

        # Game Type Dropdown
        self.game_var = tk.StringVar()
        self.session_var = tk.StringVar()

        tk.Label(self, text="Game Type:").grid(row=0, column=0, padx=10, pady=10)
        self.game_menu = ttk.Combobox(self, textvariable=self.game_var, state="readonly")
        self.game_menu['values'] = self.get_game_types()
        self.game_menu.grid(row=0, column=1)
        self.game_menu.bind("<<ComboboxSelected>>", self.update_sessions)

        # Session Dropdown
        tk.Label(self, text="Session:").grid(row=1, column=0, padx=10, pady=10)
        self.session_menu = ttk.Combobox(self, textvariable=self.session_var, state="readonly")
        self.session_menu.grid(row=1, column=1)

        # Buttons
        tk.Button(self, text="View Summary", command=self.view_summary).grid(row=2, column=0, padx=10, pady=10)
        tk.Button(self, text="View Entropy Plot", command=self.view_plot).grid(row=2, column=1, padx=10, pady=10)
        tk.Button(self, text="Export Session", command=self.export_session).grid(row=2, column=2, padx=10, pady=10)

        # Text Viewer
        self.output_box = tk.Text(self, width=90, height=15)
        self.output_box.grid(row=3, column=0, columnspan=3, padx=10, pady=10)

    def get_game_types(self):
        types = []
        for folder in os.listdir(RESULTS_DIR):
            path = os.path.join(RESULTS_DIR, folder)
            if os.path.isdir(path):
                types.append(folder)
        return types

    def update_sessions(self, event=None):
        game_type = self.game_var.get()
        session_path = os.path.join(RESULTS_DIR, game_type)
        sessions = [s for s in os.listdir(session_path) if os.path.isdir(os.path.join(session_path, s))]
        sessions.sort(reverse=True)
        self.session_menu['values'] = sessions
        if sessions:
            self.session_var.set(sessions[0])
        else:
            self.session_var.set("")

    def get_selected_path(self):
        game = self.game_var.get()
        session = self.session_var.get()
        if not game or not session:
            messagebox.showwarning("Missing Selection", "Please select a game type and session.")
            return None
        return os.path.join(RESULTS_DIR, game, session)

    def view_summary(self):
        path = self.get_selected_path()
        if not path:
            return
        try:
            with open(os.path.join(path, "summary.txt"), "r") as f:
                content = f.read()
            self.output_box.delete("1.0", tk.END)
            self.output_box.insert(tk.END, content)
        except Exception as e:
            messagebox.showerror("Error", f"Could not read summary.txt:\n{e}")

    def view_plot(self):
        path = self.get_selected_path()
        if not path:
            return
        try:
            csv_path = os.path.join(path, "entropy_log.csv")
            if not os.path.exists(csv_path):
                messagebox.showinfo("No Entropy File", "No entropy_log.csv found for this session.")
                return

            times, entropies = [], []
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    times.append(float(row[0]))
                    entropies.append(float(row[1]))

            plt.figure(figsize=(10, 4))
            plt.plot(times, entropies, label="Entropy")
            plt.title("Entropy over Time")
            plt.xlabel("Time (s)")
            plt.ylabel("Entropy")
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        except Exception as e:
            messagebox.showerror("Plot Error", f"Could not display plot:\n{e}")

    def export_session(self):
        path = self.get_selected_path()
        if not path:
            return
        export_path = filedialog.asksaveasfilename(defaultextension=".zip", filetypes=[("Zip File", "*.zip")])
        if not export_path:
            return
        try:
            with zipfile.ZipFile(export_path, 'w') as zipf:
                for file in os.listdir(path):
                    zipf.write(os.path.join(path, file), arcname=file)
            messagebox.showinfo("Export Successful", f"Session exported to:\n{export_path}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not export session:\n{e}")

if __name__ == "__main__":
    app = Dashboard()
    app.mainloop()
