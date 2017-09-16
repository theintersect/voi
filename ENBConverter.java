import java.util.*;

public class ENBConverter {
    static final String[] ALL_WORDS = {"All", "Everyone", "Team"};
    static final List<String> ALL_LIST = Arrays.asList(ALL_WORDS);
    static final String[] FIRST_NAMES = {"Howard", "Andy", "Stephen", "Andey", "Chester", "Wilton", "Tejas","Shivani","Kelly","Michael"};    
    static final List<String> FIRST_LIST =  Arrays.asList(FIRST_NAMES);    
    static final String[] MONTH_ARRAY = {"","January","February","March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
    static final int[] DAY_MONTH = {0,0,31,60,91,121,152,182,213,244,274,305,335}; //
    static final String[] DAY_WEEK = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    static final String[] WORK = {"Hardware", "Software", "Electrical", "Outreach", "General", "Rsd", "Competition", "Social", "Mentorship"};
    static final List<String> WORK_LIST = Arrays.asList(WORK);
    static String workDone;
    static final Map<String, String> WORK_MAP = new HashMap<String, String>(){
        {
            put("Button", "Button Pusher - SESSION #XX");
            put("Software", "Software - SESSION #XX");
            put("Electrical", "(Put title here)");
            put("Intake", "Intake Mechanism - SESSION #XX");
            put("Drive", "Drive Train - SESSION #XX");
            put("General", "Weekly Meeting #XX");
            put("Electrical", "(Put title here)");
            put("Rsd", "(Put title here)");
            put("Competition", "(Put title here)");
            put("Social", "(Put title here)");
            put("Mentorship", "(Put title here)");
        }
    };
    public static void main(String[] args) {
        
        Scanner console = new Scanner(System.in);
         //ignore time stamp
        String nextThing;
        do{
            nextThing = console.next();
        }while (!isLetter(nextThing.charAt(0)));
        System.out.println();
        String writtenBy = capitalize(nextThing);
        String date = getDateTime(console);
        String membersPresent = formatMembers(getMembersPresent(console));
        System.out.println(WORK_MAP.get(workDone));
        System.out.println(date);
        System.out.println("Coach Gary’s House (Fremont, CA)");
        System.out.println("Attendance: " + membersPresent);
        System.out.println("Written by: " + writtenBy);
        System.out.println();
        System.out.println("Objectives: ");
        System.out.println("Actions: ");
        System.out.println("Reflection: ");
        System.out.println("To Do: ");
        
    }
    public static String getDateTime(Scanner console){
        String nextThing;
        do{
            nextThing = console.next();
        }while (!isNumber(nextThing.charAt(0)));
        String[] startDateArray = nextThing.split("/");
        int[] startDateNumbers = new int[startDateArray.length];
        for(int i = 0; i < startDateNumbers.length; i++){
            startDateNumbers[i] = Integer.parseInt(startDateArray[i]);
        }
        int dayOfYear = DAY_MONTH[startDateNumbers[0]]+startDateNumbers[1];
        String dayOfWeek = DAY_WEEK[(dayOfYear+4)%7];
        String month = MONTH_ARRAY[startDateNumbers[0]];
        String startTime = formatTime(console.next());
        console.next(); // skip second date
        String endTime = formatTime(console.next());
        return dayOfWeek + ", " + month + " " + startDateNumbers[1] + ", " + startDateNumbers[2] + " " + startTime + " - " + endTime;
    }
    public static String formatTime(String time){
        String[] timeArray = time.split(":");
        int[] timeNumbers = new int[timeArray.length];
        for(int i = 0; i < timeNumbers.length; i++){
            timeNumbers[i] = Integer.parseInt(timeArray[i]);
        }
        String suffix = "AM";
        String minutes = Integer.toString(timeNumbers[1]);
        if (minutes.equals("0")) minutes += 0;
        if (timeNumbers[0] > 12)
            suffix = "PM";
        return timeNumbers[0]%12 + ":" + minutes + " " + suffix;
    }
    public static String formatMembers(List<String> members){
        String memberString = "";
        for(int i = 0; i < members.size()-1; i++){
            memberString += members.get(i) + ", ";
        }
        if (!members.isEmpty()){
            memberString += members.get(members.size() -1);
        }
        return memberString;
    }
    public static List<String> getMembersPresent(Scanner console){
        List<String> names = new ArrayList<>();
        String tempName = console.next();
        boolean reverse = false;
        if (ALL_LIST.contains(capitalize(tempName))){
            reverse = true;
        }
        while(!WORK_LIST.contains(tempName)){
            if (firstName(tempName) != "")
                names.add(firstName(tempName));
            tempName = console.next();
        };   
        
        if (reverse){
            List<String> tempList = new ArrayList<>();
            tempList.addAll(FIRST_LIST);
            tempList.removeAll(names);
            return tempList;
        }
        workDone = capitalize(tempName);
        if (workDone.equals("Hardware"))
            workDone = capitalize(console.next());
        return names;
    }
    public static String capitalize(String name){
        name = name.toLowerCase();
        return name.substring(0,1).toUpperCase() + name.substring(1,name.length());
    }
    
    public static String firstName(String name){
        name = capitalize(name);
        for(int i = Math.min(7, name.length()); i >=4; i--){
            if (FIRST_LIST.contains(name.substring(0,i))){
                return name.substring(0,i);
            }
        }
        return "";
    }
    public static boolean isNumber(char letter){
        return letter >= 48 && letter <= 57;
    }
    public static boolean isLetter(char letter){
        return (letter >= 65 && letter <= 90) || (letter >=97 && letter <= 122);
    }
}

