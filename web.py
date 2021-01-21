import sys
from selenium import webdriver
import os
import logging
import logging.config

# 自動ログイン関数を宣言
#
#
def AutoLogin():
  # 起動するブラウザを宣言します
  browser = webdriver.Chrome(executable_path='/usr/bin/google-chrome')
  # ログイン対象のWebページURLを宣言します
  url = "https://web-auth.akashi.ac.jp"
  # 対象URLをブラウザで表示します。
  browser.get(url)
  # ログインIdとパスワードの入力領域を取得します。
  login_id = browser.find_element_by_xpath('input name="name"')
  login_pw = browser.find_element_by_xpath('input name="pass"')
  # ログインIDとパスワードを入力します。
  userid = "e1904"
  userpw = "akashi6814"
  login_id.send_keys(userid)
  login_pw.send_keys(userpw)
  # ログインボタンをクリックします。
  login_btn = browser.find_element_by_xpath('input type="submit"')
  login_btn.click()

# AutoLogin関数を実行します。
#
ret = AutoLogin()
