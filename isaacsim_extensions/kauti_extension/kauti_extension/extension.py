import omni.ext
import omni.ui as ui
import carb
import datetime

class KautiExtensionTest(omni.ext.IExt):
    """Kauti's test extension to verify setup"""
    
    def on_startup(self, ext_id):
        carb.log_info("[Kauti Extension] Starting up...")
        
        # Initialize counter
        self._click_count = 0
        
        # Create a window
        self._window = ui.Window("Kauti Extension Test", width=400, height=300)
        
        with self._window.frame:
            with ui.VStack(spacing=10):
                # Title
                ui.Label("<� Kauti Extension Loaded Successfully!", 
                        style={"font_size": 20, "color": 0xFF00FF00})
                
                ui.Separator(height=5)
                
                # Status section
                with ui.HStack():
                    ui.Label("Status:", width=80)
                    self._status_label = ui.Label("Extension is running", 
                                                 style={"color": 0xFF88FF88})
                
                # Time display
                with ui.HStack():
                    ui.Label("Started at:", width=80)
                    self._time_label = ui.Label(
                        datetime.datetime.now().strftime("%H:%M:%S")
                    )
                
                ui.Separator(height=10)
                
                # Interactive section
                ui.Label("Test Controls:", style={"font_size": 16})
                
                # Click counter
                self._counter_label = ui.Label(f"Button clicked: {self._click_count} times")
                
                # Test buttons
                with ui.HStack(spacing=5):
                    ui.Button("Click Me!", height=40, width=120).set_clicked_fn(
                        self._on_test_click
                    )
                    ui.Button("Reset", height=40, width=120).set_clicked_fn(
                        self._on_reset_click
                    )
                
                # Color test button
                self._color_button = ui.Button("Change My Color", height=40)
                self._color_button.set_clicked_fn(self._on_color_click)
                
                ui.Spacer(height=10)
                
                # Info button
                ui.Button("Show Info", height=30).set_clicked_fn(
                    self._show_info
                )
    
    def _on_test_click(self):
        """Handle test button click"""
        self._click_count += 1
        self._counter_label.text = f"Button clicked: {self._click_count} times"
        carb.log_info(f"[Kauti Extension] Button clicked {self._click_count} times")
    
    def _on_reset_click(self):
        """Reset the counter"""
        self._click_count = 0
        self._counter_label.text = f"Button clicked: {self._click_count} times"
        carb.log_info("[Kauti Extension] Counter reset")
    
    def _on_color_click(self):
        """Change button color"""
        import random
        colors = [0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFFFFFF00, 0xFFFF00FF]
        new_color = random.choice(colors)
        self._color_button.style = {"background_color": new_color}
        carb.log_info(f"[Kauti Extension] Changed color to {hex(new_color)}")
    
    def _show_info(self):
        """Show extension information"""
        info_text = """
Kauti Extension Test v1.0.0
        
This is a test extension to verify:
 Extension loading
 UI creation
 Event handling
 Logging system
        
Everything is working correctly!
        """
        carb.log_info("[Kauti Extension] Info displayed")
        
        # Create a popup window
        self._info_window = ui.Window("Extension Info", width=300, height=200)
        with self._info_window.frame:
            ui.Label(info_text, style={"font_size": 12}, word_wrap=True)
    
    def on_shutdown(self):
        carb.log_info("[Kauti Extension] Shutting down...")
        
        # Clean up windows
        if hasattr(self, '_window') and self._window:
            self._window.destroy()
            self._window = None
            
        if hasattr(self, '_info_window') and self._info_window:
            self._info_window.destroy()
            self._info_window = None