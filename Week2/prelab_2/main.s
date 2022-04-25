    THUMB
    AREA    DATA, ALIGN=4 
                
    AREA    |.text|, CODE, READONLY, ALIGN=2  
    EXPORT  main
           
main    MOV R1, #6       
        MOV R2, #0        
        MOV R0, #0        
        MOV R4, #1
loop    CMP R0,R1
        BHS done
        AND R5,R0,R4
        CMP R5,R4
        BHS loop2
        ADD R2,R2,R0
        ADD R0,R0,#1
        B loop        
loop2   
        ADD R0,R0,#1
        B loop
 
    
done
    ALIGN      
    END