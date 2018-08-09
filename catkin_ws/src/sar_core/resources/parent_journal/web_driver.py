# import os
# from selenium import webdriver
# from pyvirtualdisplay import Display

# display = Display(visible=0, size=(800, 600))
# display.start()
# driver = webdriver.Chrome('/usr/bin/chromedriver')
# driver.get("http://chienminghuang.info")
# #print driver.page_source.encode('utf-8')
# print driver.title
# #driver.quit()
# #display.stop()
import os
from selenium import webdriver

chromedriver = "/usr/bin/chromedriver"
os.environ["webdriver.chrome.driver"] = chromedriver
driver = webdriver.Chrome(chromedriver)
driver.set_window_position(500, 100)
driver.set_window_size(960, 768)
driver.get("http://robotics.usc.edu/~expeditions")
driver.quit()