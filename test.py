# coding:utf-8
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
import time
# ブラウザを開く。
driver = webdriver.Chrome(executable_path='/usr/bin/google-chrome')
# Googleの検索TOP画面を開く。
driver.get("https://www.yahoo.co.jp/")
# 3秒待機
time.sleep(3)
# ログインボタンをクリックする
login_btn = driver.find_element_by_xpath('//*[@id="Login"]/div/p[1]/a')
login_btn.click()
# 1秒待機
time.sleep(1)
# ログインIDを入力
login_id = driver.find_element_by_name("login")
login_id.send_keys("ログインIDを入力")
# 次へボタンをクリック
next_btn = driver.find_element_by_name("btnNext")
next_btn.click()
# 1秒待機
time.sleep(1)
# パスワードを入力
password = driver.find_element_by_name("passwd")
password.send_keys("パスワードを入力")
#ログインボタンをクリック
login_btn = driver.find_element_by_name("btnSubmit")
login_btn.click()
#10秒待機
time.sleep(10)
# ブラウザを終了する。
driver.close()
