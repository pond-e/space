from selenium import webdriver

def sample(d):
    d.get('https://www.google.com')


if __name__ == '__main__':
    """
    コマンドラインからの呼出し
    """
    d = webdriver.Chrome(executable_path='/usr/bin/google-chrome')
    sample(d)
