choice = input("Select camera (1: Logitech, 2: Basler): ")

if choice == '1':
    import logitech
elif choice == '2':
    import basler
else:
    print("Invalid choice")
